/* ************************************************************************** */
/*                                                                            */
/*                                                        :::      ::::::::   */
/*   get_next_line.c                                    :+:      :+:    :+:   */
/*                                                    +:+ +:+         +:+     */
/*   By: youskim <youskim@student.42seoul.kr>       +#+  +:+       +#+        */
/*                                                +#+#+#+#+#+   +#+           */
/*   Created: 2022/01/07 22:56:11 by youskim           #+#    #+#             */
/*   Updated: 2022/04/09 13:20:31 by youskim          ###   ########.fr       */
/*                                                                            */
/* ************************************************************************** */

#include "get_next_line.h"

static char	*ft_strdup(char **join)
{
	int		i;
	char	*return_val;

	i = 0;
	return_val = (char *)malloc(sizeof(char) * (null_strlen(*join) + 1));
	if (return_val == 0)
		return (NULL);
	while (*(*join + i) != '\0')
	{
		return_val[i] = *(*join + i);
		i++;
	}
	return_val[i] = '\0';
	free(*join);
	return (return_val);
}

static char	*rd_zero(char **str_2, char **result_2, int *check, char **join)
{
	char		*return_value;

	if (gnl_strchr(*join, '\n') == 1)
	{
		return_value = str_cpy(*join);
		free(*result_2);
		*result_2 = str_cut(*join);
		free(*str_2);
		free(*join);
		return (return_value);
	}
	else
	{
		*check = 1;
		free(*str_2);
		free(*result_2);
		if (null_strlen(*join) == 0)
		{
			free(*join);
			return (NULL);
		}
		return (*join);
	}
}

static char	*rd_not_zero(char **str_1, char **result_1, char **join_1)
{
	char	*temp_1;

	temp_1 = str_cpy(*join_1);
	free(*result_1);
	*result_1 = str_cut(*join_1);
	free(*str_1);
	free(*join_1);
	return (temp_1);
}

static char	*str_set(char *str, int fd, int *rd_size)
{
	size_t	i;

	i = 0;
	while (i < BUFFER_SIZE)
	{
		str[i] = 0;
		i++;
	}
	*rd_size = read(fd, str, BUFFER_SIZE);
	str[BUFFER_SIZE] = '\0';
	return (str);
}

char	*get_next_line(int fd)
{
	char		*str;
	static char	*result = NULL;
	int			rd_size;
	static int	check = 0;
	char		*join;

	if (check == 1 || fd < 0 || fd > OPEN_MAX)
		return (NULL);
	str = (char *)malloc(sizeof(char) * (BUFFER_SIZE + 1));
	if (str == 0)
		return (NULL);
	join = str_join(result, str_set(str, fd, &rd_size));
	while (rd_size > 0)
	{
		if (gnl_strchr(join, '\n') == 1)
			return (rd_not_zero(&str, &result, &join));
		free(result);
		result = ft_strdup(&join);
		join = str_join(result, str_set(str, fd, &rd_size));
	}
	if (rd_size == 0)
		return (rd_zero(&str, &result, &check, &join));
	free(join);
	free(str);
	return (NULL);
}
