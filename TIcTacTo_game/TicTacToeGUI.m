function TicTacToeGUI
    fig = figure('Position', [500 500 400 400], 'MenuBar', 'none', 'Name', 'Tic Tac Toe', 'NumberTitle', 'off', 'Resize', 'off');
    set(fig, 'Color', [0.6 0.8 1]);
    btnSize =134;
    boardButtons = zeros(3, 3);
    for i = 1:3
        for j = 1:3
            boardButtons(i,j) = uicontrol('Style', 'pushbutton', 'Position', [(j-1)*btnSize (3-i)*btnSize btnSize btnSize], ...
                'Callback', @(src, event) buttonCallback(src, i, j));
        end
    end
    gameState = zeros(3, 3);
    currentPlayer = coinToss();
    msgbox(sprintf('Player %d goes first!', currentPlayer), 'Coin Toss Result');
    function player = coinToss()
        player = randi([1,2]);
    end
    function buttonCallback(src, i, j)
        if gameState(i, j) == 0
            gameState(i, j) = currentPlayer;
            set(src, 'String', char('X' - 1 + currentPlayer), 'Enable', 'off');
            set(fig, 'Color', [0.6 0.8 1]);
            if checkWin(gameState, currentPlayer)
                msgbox(sprintf('Player %d wins! Congratulation!', currentPlayer), 'Game over');
                resetGame;
            elseif all(gameState(:) ~= 0)
                msgbox('This game is a draw!, Better luck next time:)', 'Game Over');
                resetGame;
            else
                currentPlayer = 3 - currentPlayer;
            end
        end
    end
    function isWin = checkWin(board, player)
        isWin = any(all(board == player, 2)) || any(all(board == player, 1)) || ...
            all(diag(board) == player) || all(diag(flipud(board)) == player);
    end
    function resetGame
        set(boardButtons, 'String', '', 'Enable', 'on');
        gameState = zeros(3, 3);
        currentPlayer = coinToss(); 
        msgbox(sprintf('Player %d goes first!', currentPlayer), 'Coin Toss Result');
    end
end
